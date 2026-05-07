#!/bin/sh
cd `dirname $0`

MODULE=$(basename "$PWD")
export PATH=$PATH:$(go env GOPATH)/bin

if ! (
    go get go.viam.com/rdk@latest > /dev/null 2>&1
    go mod tidy > /dev/null 2>&1
); then
    echo "Go packages could not be installed. Quitting..." >&2
    exit 1
fi

go build -o bin/$MODULE main.go
exec bin/$MODULE $@
