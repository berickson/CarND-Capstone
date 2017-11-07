# kills the most recently started docker container
CID=$(docker ps --latest --quiet)
docker kill  $CID