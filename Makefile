.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

.PHONY: build
build:
	make dspace
	make foxglove

.PHONY: dspace
dspace:
	docker build -t iac_sut_te_bridge:iron -f Dockerfile . --target sut-te-bridge_dev

.PHONY: foxglove
foxglove:
	docker build -t iac_sut_te_bridge_foxglove:iron -f Dockerfile . --target sut-te-bridge_foxglove

.PHONY: simphera
simphera:
	docker build -t iac_sut_te_bridge_simphera:iron -f Dockerfile . --target sut-te-bridge_simphera
