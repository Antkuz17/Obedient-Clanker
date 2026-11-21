# ArduPilot Copter SITL (official prebuilt stable binary). Exposes MAVLink on TCP 5760.
FROM --platform=linux/amd64 debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /sitl
RUN curl -fsSL -o arducopter https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter \
    && chmod +x arducopter \
    && curl -fsSL -o copter.parm https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params/copter.parm
EXPOSE 5760
# Home = Western University. --speedup 1 so wall-clock timeouts match.
ENTRYPOINT ["./arducopter", "--model", "+", "--defaults", "copter.parm", \
            "--home", "43.0096,-81.2737,250,0", "--speedup", "1", "-I0"]
