FROM espressif/idf:release-v4.4

# Install protocol buffers
RUN apt update && apt install -y protobuf-compiler

# Install pre-commit and requirements for building protobuf messages
RUN pip install pre-commit protobuf grpcio-tools

# Pre-install the hooks that pre-commit needs. The contents of the .git/hooks
# folder will get overwritten by the volume mount, but the executables for the
# hooks will be there!

COPY .pre-commit-config.yaml /code/.pre-commit-config.yaml
WORKDIR /code
RUN git init . && pre-commit install-hooks

# Make sure idf.py is added to PATH even when running through VS Code
RUN echo "source /opt/esp/idf/export.sh" >> /root/.bashrc
