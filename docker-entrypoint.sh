#!/bin/sh
set -e

PORT=${PORT:-8080}

sed -i "s/listen 80;/listen ${PORT};/g" /etc/nginx/conf.d/default.conf

cat > /usr/share/nginx/html/config.js <<EOF
// Runtime configuration injected at container startup
window.env = window.env || {};
EOF

if grep -q '<script src="/config.js"></script>' /usr/share/nginx/html/index.html; then
  echo "config.js already injected in index.html"
else
  sed -i 's|</head>|  <script src="/config.js"></script>\n  </head>|' /usr/share/nginx/html/index.html
fi

exec nginx -g "daemon off;"
