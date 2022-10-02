# Testing branch
## Additional reqs compared to main

```
sudo apt-get -y install libxml2-utils
```

## Checkout
(assumes that you have checked out MowgliRover in ~/MowgliRover)

```
cd ~/MowgliRover
git checkout testing
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r -y
```

Then recompile with ./scripts/build_all.sh
