Collector Agent
---------------

## Nodes

There are two nodes:

* *applicant* - sends a request to issue new certificates based on the log in the objective field
* *collector* - continuously collect the data from the solar panel

## Service

*make_demand* - call to create a demand message

## Build

```
nix build -f release.nix
```

## Run

```
source result/setup.zsh
roslaunch collector-agent applicant.launch \
    user:=<postgres_user_name> \
    password_file:=<path_to_password_file> \
    rest_api_keys_file:=<path_to_rest_api_user_and_password_file>
```

```
roslaunch collector-agent collector.launch \
    user:=<postgres_user_name> \
    password_file:=<path_to_password_file> \
    rest_api_keys_file:=<path_to_rest_api_user_and_password_file>
```

By default, `host = 127.0.0.1`, `database = skolkovo` and `port = 5432`

