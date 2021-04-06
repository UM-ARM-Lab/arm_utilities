import pathlib
from typing import Optional, Iterable

from colorama import Fore

from arc_utilities.path_utils import rm_tree


def mkdir_and_ask(path, parents: bool, yes: Optional[bool] = False):
    if path.exists():
        msg = f"Path {path} exists, do you want to reuse it? [Y/n]"
        if yes:
            print(f"{msg} answering yes")
            return True
        else:
            response = input(msg)
            if response == 'y' or response == 'Y' or response == '':
                return True
            else:
                return False

    path.mkdir(parents=parents, exist_ok=False)
    return True


def get_all_subfolders(args):
    all_subfolders = []
    for results_dir in args.results_dirs:
        subfolders = results_dir.iterdir()
        for subfolder in subfolders:
            if subfolder.is_dir():
                all_subfolders.append(subfolder)
    return all_subfolders


def directory_size(dir: pathlib.Path):
    return sum(f.stat().st_size for f in dir.glob('**/*') if f.is_file())


def append_str_to_path(p: pathlib.Path, s: str):
    return p.parent / (p.name + s)


def ask_to_remove_directories(directories_to_remove: Iterable[pathlib.Path]):
    print("Ok to delete these directories?")
    for d in directories_to_remove:
        print(d.as_posix())
    k = input("[Y/n]")
    if k == '' or k == 'y' or k == 'Y':
        print(Fore.GREEN + "Deleting.")
        for d in directories_to_remove:
            rm_tree(d)

        return

    print(Fore.RED + "Aborting.")
    return



def count_files_recursive(path):
    count = 0
    path = pathlib.Path(path)
    for child in path.iterdir():
        if child.is_dir():
            count += count_files_recursive(child)
        else:
            count += 1
    return count
