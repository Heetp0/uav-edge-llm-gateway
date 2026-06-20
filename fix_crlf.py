import os

repo_dir = os.path.dirname(os.path.abspath(__file__))
extensions = {'.py', '.sh', '.yaml', '.xml', '.txt', '.md'}

for root, _, files in os.walk(repo_dir):
    # Skip .git directory
    if '.git' in root:
        continue
    for f in files:
        ext = os.path.splitext(f)[1]
        if ext in extensions:
            path = os.path.join(root, f)
            with open(path, 'rb') as file:
                content = file.read()
            if b'\r\n' in content:
                content = content.replace(b'\r\n', b'\n')
                with open(path, 'wb') as file:
                    file.write(content)
                print(f"Fixed CRLF in: {path}")
