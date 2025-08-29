COMPOSE=docker compose -f docker-compose.yml

.PHONY: build run

build:
	$(COMPOSE) build

run:
	$(COMPOSE) run --rm moveit2
