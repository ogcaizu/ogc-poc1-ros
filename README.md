# ogc-poc1-ros

## Requirements

||version|
|:--|:--|
|docker|18.03.1-ce|
|docker-compose|1.21.1|

## Usage
### build container
1. build containers

    ```bash
    $ docker-compose build
    ```

### start rosbridge with roscore
1. start roscore & rosbridge using docker-compose

    ```bash
    $ master=roscore;docker-compose up &
    ```

## start rosbridge without roscore (roscore is runnning on docker host)
1. check the gateway address of docker network

    ```bash
    $ docker-compose up roscore &
    $ export master=$(docker inspect --format='{{range .NetworkSettings.Networks}}{{.Gateway}}{{end}}' roscore);echo ${master}
    $ docker-compose down
    ```
1. start rosbridge using outside roscore

    ```bash
    $ docker-compose up rosbridge &
    ```

## stop roscore and rosbridge
1. stop docker-compose

    ```bash
    $ docker-compose down
    ```

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 [TIS Inc.](https://www.tis.co.jp/)
