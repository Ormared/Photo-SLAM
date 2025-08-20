image-build:
	docker --debug build -t photo_slam .

build:
	docker compose up --build -d

compose:
	docker compose up -d

run:
	docker compose exec photo-slam bash
