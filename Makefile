
default: deploy

.PHONY: deploy
deploy:
	cd device && cargo hf2 --release

.PHONY: test
test:
	cargo test

