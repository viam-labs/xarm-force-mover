BIN_OUTPUT_PATH = bin
MODULE_BINARY = $(BIN_OUTPUT_PATH)/xarm-force-mover

build:
	go build -o $(MODULE_BINARY) main.go

module.tar.gz: build
	tar czf $(BIN_OUTPUT_PATH)/module.tar.gz $(MODULE_BINARY) meta.json

clean:
	rm -rf $(BIN_OUTPUT_PATH)

format:
	gofmt -w -s .

setup:
	go mod tidy

update-rdk:
	go get go.viam.com/rdk@latest
	go mod tidy
