## Сборка образа для разных платформ

### Впервые

```console
docker buildx create --name <builder_name> --use --bootstrap
docker buildx build --platform linux/amd64,linux/arm64 --output type=docker . -t <image_name>
```

### В дальнейшем

```console
docker buildx use <builder_name>
docker buildx inspect --bootstrap <builder_name>
docker buildx build --platform linux/amd64,linux/arm64 --output type=docker . -t <image_name>
```

### Запуск контейнера

```console
docker compose up -d
```

### Остановка контейнера

```console
docker compose stop
```
