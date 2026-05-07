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
	Arm            string `json:"arm"`
	PollIntervalMS int    `json:"poll_interval_ms,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	return []string{cfg.Arm}, nil, nil
}

type armMover struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	arm    arm.Arm

	pollInterval time.Duration
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
		name:         rawConf.ResourceName(),
		logger:       logger,
		arm:          a,
		pollInterval: 50 * time.Millisecond,
	}
	if conf.PollIntervalMS > 0 {
		m.pollInterval = time.Duration(conf.PollIntervalMS) * time.Millisecond
	}

	return m, nil
}

func (m *armMover) Name() resource.Name { return m.name }

func (m *armMover) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return m.run(ctx, cmd)
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

	axisRaw, ok := cmd["axis"]
	if !ok {
		return nil, fmt.Errorf("axis is required in DoCommand args")
	}
	axisStr, ok := axisRaw.(string)
	if !ok {
		return nil, fmt.Errorf("axis must be a string, got %T", axisRaw)
	}
	axis := strings.ToLower(axisStr)
	switch axis {
	case "x", "y", "z":
	default:
		return nil, fmt.Errorf("axis must be one of 'x','y','z', got %q", axisStr)
	}

	targetRaw, ok := cmd["target"]
	if !ok {
		return nil, fmt.Errorf("target is required in DoCommand args")
	}
	target, ok := targetRaw.(float64)
	if !ok {
		return nil, fmt.Errorf("target must be a number, got %T", targetRaw)
	}

	startPose, err := m.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm position: %w", err)
	}
	targetPoint := startPose.Point()
	switch axis {
	case "x":
		targetPoint.X = target
	case "y":
		targetPoint.Y = target
	case "z":
		targetPoint.Z = target
	}
	targetPose := spatialmath.NewPose(targetPoint, startPose.Orientation())

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
