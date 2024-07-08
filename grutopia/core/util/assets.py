import io


def read_file(path: str) -> io.BytesIO:
    with open(path, 'rb') as f:
        return io.BytesIO(f.read())
