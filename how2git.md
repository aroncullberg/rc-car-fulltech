# Submodules ELI5

## a. Adding a submodule
```bash
git submodule add <repository> [<path>]
git commit -m "Add repo as a submodule"

```

## b. Cloning a repository with submodules
```bash
git clone --recurse-submodules <repository>
```

# c. what you(me) probably want to do
```bash
git submodule update --init --recursive
```
