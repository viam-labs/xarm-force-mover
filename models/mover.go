package models

import (
	"context"
	"fmt"
	"strings"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
)

var ArmMover = resource.NewModel("viam", "xarm-force-mover", "arm")

func init() {
	resource.RegisterService(genericservice.API, ArmMover,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newArmMover,
		},
	)
}

type Config struct {
	Arm            string  `json:"arm"`
	Axis           string  `json:"axis,omitempty"`
	Direction      string  `json:"direction,omitempty"`
	MoveDistanceMM float64 `json:"move_distance_mm,omitempty"`
	PollIntervalMS int     `json:"poll_interval_ms,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if cfg.Axis != "" {
		switch strings.ToLower(cfg.Axis) {
		case "x", "y", "z":
		default:
			return nil, nil, fmt.Errorf("%s: axis must be one of 'x','y','z', got %q", path, cfg.Axis)
		}
	}
	if cfg.Direction != "" {
		switch strings.ToLower(cfg.Direction) {
		case "positive", "negative", "+", "-":
		default:
			return nil, nil, fmt.Errorf("%s: direction must be 'positive' or 'negative', got %q", path, cfg.Direction)
		}
	}
	return []string{cfg.Arm}, nil, nil
}

type armMover struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	arm    arm.Arm

	axis           string
	signMul        float64
	moveDistanceMM float64
	pollInterval   time.Duration
}

func newArmMover(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	a, err := arm.FromProvider(deps, conf.Arm)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm %q: %w", conf.Arm, err)
	}

	m := &armMover{
		name:           rawConf.ResourceName(),
		logger:         logger,
		arm:            a,
		axis:           "z",
		signMul:        -1.0,
		moveDistanceMM: 500.0,
		pollInterval:   50 * time.Millisecond,
	}
	if conf.Axis != "" {
		m.axis = strings.ToLower(conf.Axis)
	}
	if conf.Direction != "" {
		switch strings.ToLower(conf.Direction) {
		case "positive", "+":
			m.signMul = 1.0
		case "negative", "-":
			m.signMul = -1.0
		}
	}
	if conf.MoveDistanceMM != 0 {
		m.moveDistanceMM = conf.MoveDistanceMM
	}
	if conf.PollIntervalMS > 0 {
		m.pollInterval = time.Duration(conf.PollIntervalMS) * time.Millisecond
	}

	return m, nil
}

func (m *armMover) Name() resource.Name { return m.name }

func (m *armMover) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["start"]; ok {
		return m.run(ctx, cmd)
	}
	return nil, fmt.Errorf("unknown command: expected key %q", "start")
}

func (m *armMover) run(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	jointRaw, ok := cmd["joint"]
	if !ok {
		return nil, fmt.Errorf("joint is required in DoCommand args")
	}
	jointF, ok := jointRaw.(float64)
	if !ok {
		return nil, fmt.Errorf("joint must be a number, got %T", jointRaw)
	}
	joint := int(jointF)
	if joint < 0 {
		return nil, fmt.Errorf("joint must be >= 0, got %d", joint)
	}

	startPose, err := m.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm position: %w", err)
	}
	target := startPose.Point()
	delta := m.moveDistanceMM * m.signMul
	switch m.axis {
	case "x":
		target.X += delta
	case "y":
		target.Y += delta
	case "z":
		target.Z += delta
	}
	targetPose := spatialmath.NewPose(target, startPose.Orientation())

	moveDone := make(chan error, 1)
	go func() {
		moveDone <- m.arm.MoveToPosition(ctx, targetPose, nil)
	}()

	var lastVal *float64
	ticker := time.NewTicker(m.pollInterval)
	defer ticker.Stop()

	for {
		select {
		case err := <-moveDone:
			if err != nil {
				return nil, fmt.Errorf("move completed with error before contact: %w", err)
			}
			return nil, fmt.Errorf("move completed without force sign change")
		case <-ctx.Done():
			_ = m.arm.Stop(context.Background(), nil)
			<-moveDone
			return nil, ctx.Err()
		case <-ticker.C:
		}

		val, err := m.readForce(ctx, joint)
		if err != nil {
			_ = m.arm.Stop(context.Background(), nil)
			<-moveDone
			return nil, err
		}

		if lastVal != nil && (val < 0) != (*lastVal < 0) {
			endPose, _ := m.arm.EndPosition(ctx, nil)
			if err := m.arm.Stop(context.Background(), nil); err != nil {
				return nil, fmt.Errorf("failed to stop arm: %w", err)
			}
			<-moveDone
			m.logger.Infof("Contact detected: load[%d] sign flip %f -> %f", joint, *lastVal, val)
			result := map[string]interface{}{
				"success":     true,
				"prev_force":  *lastVal,
				"final_force": val,
			}
			if endPose != nil {
				p := endPose.Point()
				result["final_x"] = p.X
				result["final_y"] = p.Y
				result["final_z"] = p.Z
			}
			return result, nil
		}
		lastVal = &val
	}
}

// readForce reads load[joint] from a ufactory xArm via DoCommand({"load": true}).
func (m *armMover) readForce(ctx context.Context, joint int) (float64, error) {
	resp, err := m.arm.DoCommand(ctx, map[string]interface{}{"load": true})
	if err != nil {
		return 0, fmt.Errorf("load DoCommand failed: %w", err)
	}
	raw, ok := resp["load"]
	if !ok {
		return 0, fmt.Errorf("load response missing 'load' key")
	}
	arr, ok := raw.([]interface{})
	if !ok {
		return 0, fmt.Errorf("load response is not an array, got %T", raw)
	}
	if joint >= len(arr) {
		return 0, fmt.Errorf("joint %d out of range (len=%d)", joint, len(arr))
	}
	f, ok := arr[joint].(float64)
	if !ok {
		return 0, fmt.Errorf("load[%d] is not a float64, got %T", joint, arr[joint])
	}
	return f, nil
}

func (m *armMover) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (m *armMover) Close(context.Context) error { return nil }
