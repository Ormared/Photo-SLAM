image-build:
	docker --debug build -t photo_slam-dev .

build:
	docker compose up --build -d

compose:
	docker compose up -d

run:
	docker compose exec photo-slam bash

down:
	docker compose down

replica:
	env -u LIBGL_ALWAYS_INDIRECT -u LIBGL_ALWAYS_SOFTWARE -u GALLIUM_DRIVER -u MESA_GL_VERSION_OVERRIDE -u MESA_GLSL_VERSION_OVERRIDE ./bin/replica_rgbd ./ORB-SLAM3/Vocabulary/ORBvoc.txt ./cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml ./cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd.yaml ./data/Replica/office0 ./results/office0

