package main

import (
	"xarm-force-mover/models"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: genericservice.API, Model: models.ArmMover},
	)
}
