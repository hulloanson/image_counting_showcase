volumes=-v $(shell pwd):/app

catkin_ws=/app/catkin_ws

dev_image=image_counting_dev_image:latest
	
dev_docker_cmd_base=docker run --network=host --rm ${volumes} --privileged --workdir "${catkin_ws}"

dev_docker_cmd_user=${dev_docker_cmd_base} 

# --user $(shell id -u)

build_dev_image:
	docker build --network=host -t ${dev_image} ./docker


ros1_shell: build_dev_image
	${dev_docker_cmd_user} -it ${dev_image} /bin/bash

catkin:
	${dev_docker_cmd_user} ${dev_image} /bin/bash -c 'catkin_make'

clean:
	${dev_docker_cmd_user} ${dev_image} /bin/bash -c 'cd ${catkin_ws} && rm -rf build devel install'

run_publisher: build_dev_image catkin
	${dev_docker_cmd_user} -it ${dev_image} /bin/bash -c 'source devel/setup.bash; cd ${catkin_ws}/src/image_counting; echo running image_publisher...; src/image_publisher.py; echo image publisher exited with code $$? .'
	
run_counter: build_dev_image catkin
	${dev_docker_cmd_user} -it ${dev_image} /bin/bash -c 'source devel/setup.bash; echo running image counter...; ${catkin_ws}/src/image_counting/src/image_counter.py;  echo image counter exited with code $$? .'
	
run_saver: build_dev_image catkin
	${dev_docker_cmd_user} -it ${dev_image} /bin/bash -c 'source devel/setup.bash; echo running image path saver...; ${catkin_ws}/src/image_counting/src/path_saver.py;  echo image path saver exited with code $$? .'
	
run: build_dev_image catkin
	${dev_docker_cmd_user} -it ${dev_image} /bin/bash -c 'source devel/setup.bash; echo running everything; roslaunch image_counting everything.launch; echo Done running everything'
	
run_roscore:
	${dev_docker_cmd_base} ${dev_image} /bin/bash -c 'roscore'
