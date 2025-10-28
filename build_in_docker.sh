docker build -t kite-motion-planner --progress=plain .
docker run -v ./docker_build:/app/build kite-motion-planner
