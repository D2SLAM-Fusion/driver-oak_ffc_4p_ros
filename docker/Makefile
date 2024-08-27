oak_ffc_4p:
	@docker build -t hkustswarm/ros_oak_quad_cam:latest -f Dockerfile ..

ros_oak_quad_cam_dockerhub:
	@docker buildx build --platform linux/amd64,linux/arm64 -t hkustswarm/ros_oak_quad_cam:latest -f Dockerfile .. --push