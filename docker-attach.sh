# attaches terminal to docker create with docker-start.sh
# and sets environment for ros usage
CID=$(docker ps --latest --quiet)
docker exec -it $CID bash --init-file devel/setup.sh