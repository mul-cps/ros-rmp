copy from docker build to local:

```bash
docker buildx build --output type=local,dest=. .
```