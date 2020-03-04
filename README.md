# OpenRAVE-py3

This branch allows the user to build OpenRAVE, the Open Robotics Automation Virtual Environment, and its Python3 bindings on macOS and Linux.

Preparation
-----------
* Specify the installation path of `OpenRAVE` by `OPENRAVE_INSTALL_DIR`. 
 
One can assign this macro during the shell initialization, for example, by executing
```
echo 'export OPENRAVE_INSTALL_DIR='$HOME'/Projects/openrave' >> ~/.bash_profile
```
when using `bash` in macOS before Catalina, or
```
echo 'export OPENRAVE_INSTALL_DIR='$HOME'/Projects/openrave' >> ~/.zprofile
```
when using `zsh` in macOS Catalina.

In Linux, the bash configuration is initialized with `~/.bashrc` instead of `~/.bash_profile`.

* The default cmake arguments of OpenRAVE in production branch are
```
-DODE_USE_MULTITHREAD:BOOL=ON \
  -DOPT_IKFAST_FLOAT32:BOOL=OFF \
  -DNATIVE_COMPILE_FLAGS:STRING="-march=native -mtune=native" \
  -DOPT_LOG4CXX:BOOL=ON \
  -DUSE_PYBIND11_PYTHON3_BINDINGS:BOOL=ON \
  -DCMAKE_INSTALL_PREFIX=$OPENRAVE_INSTALL_DIR
```

* Use of `pybind11` instead of `Boost.Python` for Python bindings, both Python2 and Python3.
As `Boost.Python` changes its Numpy API since v1.65 and OpenRAVE's production branch uses `Boost.Python` v1.62 for Python2 bindings, we shall be using `pybind11` (https://github.com/pybind/pybind11) for both Python2/3 bindings. One can install `pybind11` with the following cmake arguments
```
-DPYBIND11_TEST=0 -DCMAKE_INSTALL_PREFIX=$OPENRAVE_INSTALL_DIR
```

Install Python3 and Boost on macOS
----------------------------------
* Install Homebrew.
```
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```
See <https://brew.sh/> for more details.

* Install third-party dependencies with Homebrew.
```
brew install libxml2 boost qt open-scene-graph
# explained below
# brew install python3 boost-python3
```

Homebrew installs these packages under `/usr/local/Cellar`, and links files in the corresponding directories where they belong:
  * executables are linked to `/usr/local/bin`,
  * header files are linked to `/usr/local/include`, 
  * libraries are linked to `/usr/local/lib`, etc.

Sometimes Homebrew installs a package as "keg-only", meaning "without linking". Then the user needs to do `brew link` to establish the linkings manually; see
https://docs.brew.sh/FAQ#what-does-keg-only-mean

* Install Python3.
One way to install Python3 is by Homebrew: the command
```
brew install python3
```
installs Python3 under `/usr/local/Cellar` and links files under `/usr/local`. 

However, a more preferrable way is using its 64-bit installer (`python-3.x.y-macosx10.9.pkg`), which one can download from https://www.python.org/downloads/mac-osx/. It installs Python3 under 
```
/Library/Frameworks/Python.framework/Versions/3.x
```
Henceforth we use Python3.8 for demostration.

Having installed Python3, we need to install Boost's Python3 module:
```
brew install boost-python3
```

Install Python3 and Boost on Linux
----------------------------------
* Install Python3.x and Boost 1.7x

We show how to build Python 3.8 and Boost 1.71 from source files. Other similar versions can be installed analogously.

```
# Install Python 3.8
wget https://www.python.org/ftp/python/3.8.0/Python-3.8.0.tgz
tar xvfz Python-3.8.0.tgz
cd Python-3.8.0
./configure --prefix=$OPENRAVE_INSTALL_DIR --enable-shared
make && make install
cd $OPENRAVE_INSTALL_DIR/bin
./pip3 install numpy ipython
```

```
# Install Boost 1.71 
wget https://dl.bintray.com/boostorg/release/1.71.0/source/boost_1_71_0.tar.gz
tar xvfz boost_1_71_0.tar.gz
cd boost_1_71_0
bash bootstrap.sh --prefix=$OPENRAVE_INSTALL_DIR --with-python=$OPENRAVE_INSTALL_DIR/bin/python3
./b2 --prefix=$OPENRAVE_INSTALL_DIR install
```

Install OpenRAVE-py3
--------------------
* Add search paths.
Add the subdirectory `lib` of `$OPENRAVE_INSTALL_DIR` into the library search paths by
```
# assume macOS Catalina; please modify accordingly
echo 'export LD_LIBRARY_PATH='$OPENRAVE_INSTALL_DIR/lib':$LD_LIBRARY_PATH'     >> ~/.zprofile
echo 'export DYLD_LIBRARY_PATH='$OPENRAVE_INSTALL_DIR/lib':$DYLD_LIBRARY_PATH' >> ~/.zprofile
```
In macOS, we add Python3 binary directory into `$PATH`.
```
export PATH="/Library/Frameworks/Python.framework/Versions/3.8/bin:$PATH"
```
Add also the Python3 `site-packages` directory inside `$OPENRAVE_INSTALL_DIR` into `$PYTHONPATH`:
```
export PYTHONPATH="$OPENRAVE_INSTALL_DIR/lib/python3.8/site-packages:$PYTHONPATH"
```

* Clone this repository and check out this `macOS` branch.
```
git clone git@github.com:rdiankov/openrave.git
git checkout macOS
```

In `CMakeLists.txt`, one edits accordingly Python3's version, Boost's version, and installation paths and version numbers of other third-party dependencies. (G Tan: to be written ...)

* Build OpenRAVE and its Python3 bindings.
```
cd openrave
mkdir build
cd build
cmake -DODE_USE_MULTITHREAD:BOOL=ON \
  -DOPT_IKFAST_FLOAT32:BOOL=OFF \
  -DNATIVE_COMPILE_FLAGS:STRING="-march=native -mtune=native" \
  -DOPT_LOG4CXX:BOOL=ON \
  -DUSE_PYBIND11_PYTHON3_BINDINGS:BOOL=ON \
  -DCMAKE_INSTALL_PREFIX=$OPENRAVE_INSTALL_DIR ..
make -j4
make install
```

License
-------
* LGPL <https://www.gnu.org/licenses/license-list.html#LGPL>
