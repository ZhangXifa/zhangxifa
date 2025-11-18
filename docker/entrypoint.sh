#!/bin/sh
set -e

echo "[entrypoint] working dir: $(pwd)"

# 确保点云持久化目录存在
mkdir -p /app/point_clouds/temp || true
mkdir -p /app/point_clouds || true

# 如果提供了环境变量，则生成 mysql.ini
if [ -n "${MYSQL_HOST}" ]; then
  echo "[entrypoint] generating mysql.ini from env"
  : "${MYSQL_PORT:=3306}"
  : "${MYSQL_USER:=root}"
  : "${MYSQL_PASSWORD:=your_password}"
  : "${MYSQL_DBNAME:=zhangxifa}"
  : "${MYSQL_INIT_SIZE:=10}"
  : "${MYSQL_MAX_SIZE:=1024}"
  : "${MYSQL_MAX_IDLE_TIME:=60}"
  : "${MYSQL_CONN_TIMEOUT_MS:=100}"
  cat > /app/mysql.ini <<EOF
#数据库连接池的配置文件(由容器启动时生成)
ip=${MYSQL_HOST}
port=${MYSQL_PORT}
username=${MYSQL_USER}
password=${MYSQL_PASSWORD}
dbname=${MYSQL_DBNAME}
initSize=${MYSQL_INIT_SIZE}
maxSize=${MYSQL_MAX_SIZE}
maxIdleTime=${MYSQL_MAX_IDLE_TIME}
connectionTimeOut=${MYSQL_CONN_TIMEOUT_MS}
EOF
fi

# 强制在容器中以前台模式运行
if [ -f /app/nginx.conf ]; then
  echo "[entrypoint] forcing Daemon=0 for container"
  sed -i -E 's/^(Daemon\s*=\s*)[01]/\10/' /app/nginx.conf || true
fi

# 日志文件位置提示
echo "[entrypoint] log file per nginx.conf: error.log (relative to /app)"

echo "[entrypoint] starting server..."
exec /app/build/bin/nginx