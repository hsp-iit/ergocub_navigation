 #!/bin/bash         
cd $PWD
docker build . --build-arg "GIT_USERNAME=$1" --build-arg "GIT_USER_EMAIL=$2" -t ergocub_navigation:latest -f Dockerfile
