Collector Service 
-----------------

**model:** `QmUB6ajZTLLMZg7re1v4hw44aoG8HDQDHr9JyujU264Aw2`

This agent is responsible for collecting data from renewable energy source and publishing a demand message.

Have a look at the [Issuing service](https://github.com/DAO-IPCI/issuing-service-package) to see the second part of the project.

## Nodes

There are two nodes:

* *applicant* - packs all the collected data to `objective` and sends a demand message to issue new certificates 
* *collector* - continuously collects the data from the solar panel

## Services

* *publish_demand* - creates a demand message and publishes to Robonomics network
* *get_objective* - packs all the data to a rosbag file, publishes to IPFS network and returns the IPFS hash of the file

## Build

```
nix build -f release.nix
```

## Run

```
roslaunch collector_agent applicant.launch db_url_config:=./db_url.config rest_api_key_config:=./rest_api_key.config
```

Where 

* `db_url_config` - a file contains a string that indicates database dialect and connection arguments in [SQLAlchemy](https://docs.sqlalchemy.org/en/13/core/engines.html?highlight=create_engine#sqlalchemy.create_engine) form
* `rest_api_key_config` - a file contains a string that indicates a username/password for REST API server of energy source

It's possible to specify a table name for the database. By default it's `solar`

## Proxy

There is a small proxy server called `proxy.py`. It's purpose is to call ROS services from POST requests. The default port is `8899`

* `/` - calls `publish_demand` service
* `/get_objective` - calls `get_objective` service

