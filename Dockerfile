FROM emscripten/emsdk:latest AS wasm-builder
WORKDIR /src
COPY src/physics/ .
RUN mkdir -p output
RUN emcc engine.cpp -o output/physics.js \
  -lembind \
  -s MODULARIZE=1 \
  -s EXPORT_NAME='createPhysicsModule' \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s ENVIRONMENT=web \
  -O3

FROM node:20-alpine AS builder

WORKDIR /app

ARG VITE_API_URL=http://localhost:8080
ENV VITE_API_URL=$VITE_API_URL

COPY package*.json ./

RUN npm ci --only=production=false

COPY . .

COPY --from=wasm-builder /src/output/physics.js ./public/wasm/physics.js
COPY --from=wasm-builder /src/output/physics.wasm ./public/wasm/physics.wasm

RUN npm run build

FROM nginx:alpine

COPY --from=builder /app/dist /usr/share/nginx/html

COPY nginx.conf /etc/nginx/conf.d/default.conf

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

EXPOSE 8080

ENTRYPOINT ["/docker-entrypoint.sh"]
