# starts a single docker for capstone project
# 
# note: only run one at a time, there is no guard
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ -d -it capstone
