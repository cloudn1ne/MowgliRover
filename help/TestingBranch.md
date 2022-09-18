# Testing branch

(assumes that you have checked out MowgliRover in ~/MowgliRover)

```
cd ~/MowgliRover
git checkout testing
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r -y
```

Then recompile with ./scripts/build_all.sh
