Collector Agent
---------------

## Nodes

There are two nodes:

* *trader_node* - to work on offchain negotiations process
* *worker_node* - trivial liability process executor

## Parameters

Objective `QmVAFgUxBitKqtV2sjaYcHkKfcAPVy3GswhaE5n5bcgLkf`

## Build

```
nix build -f release.nix
```

## Run

It's important to specify the `keyfile` file with a private key

```
source result/setup.zsh
roslaunch collector-agent worker.launch keyfile:=/root/collector-agent/keyfile

roslaunch collector-agent trader.launch
```

