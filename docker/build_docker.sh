 #!/bin/bash         
cd $PWD
docker build . --build-arg "GIT_USERNAME=$1" --build-arg "GIT_USER_EMAIL=$2" -t ergocub_base_v1:latest -f Dockerfile
