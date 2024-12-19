.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

.PHONY: vcs-import
vcs-import:
	@VCS_FILE="${VCS_FILE}"
	vcs import < ${VCS_FILE}

.PHONY: build
build:
	make simphera
	make foxglove

.PHONY: build-no-cache
build-no-cache:
	make simphera-no-cache
	make foxglove-no-cache

.PHONY: dev
dev:
	docker build -t iac_sut_te_bridge_dev:iron -f Dockerfile . --target sut-te-bridge_dev

.PHONY: dev-no-cache
dev-no-cache:
	docker build -t iac_sut_te_bridge_dev:iron -f Dockerfile . --target sut-te-bridge_dev --no-cache

.PHONY: foxglove
foxglove:
	docker build -t iac_sut_te_bridge_foxglove:iron -f Dockerfile . --target sut-te-bridge_foxglove

.PHONY: foxglove-no-cache
foxglove-no-cache:
	docker build -t iac_sut_te_bridge_foxglove:iron -f Dockerfile . --target sut-te-bridge_foxglove --no-cache

.PHONY: simphera
simphera:
	docker build -t iac_sut_te_bridge:iron -f Dockerfile . --target sut-te-bridge_simphera

.PHONY: simphera-no-cache
simphera-no-cache:
	docker build -t iac_sut_te_bridge:iron -f Dockerfile . --target sut-te-bridge_simphera --no-cache

.PHONY: race_msgs
race_msgs:
	make vcs-import VCS_FILE=race_msgs.repos
