ignore_list = ['.vscode', 'build', 'package', '.git', '.idea', '.vs', 'cmake-build-debug']

def is_ignored(item):
    return item in ignore_list

