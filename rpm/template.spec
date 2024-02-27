%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

Name:           ros-humble-dbw-ford
Version:        2.1.10
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS dbw_ford package

License:        BSD
URL:            http://dataspeedinc.com
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-humble-dbw-ford-can
Requires:       ros-humble-dbw-ford-description
Requires:       ros-humble-dbw-ford-joystick-demo
Requires:       ros-humble-dbw-ford-msgs
Requires:       ros-humble-ros-workspace
BuildRequires:  ros-humble-ament-cmake
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
Drive-by-wire interface to the Dataspeed Inc. Lincoln MKZ DBW kit

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/humble" \
    -DAMENT_PREFIX_PATH="/opt/ros/humble" \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Tue Feb 27 2024 Kevin Hallenbeck <khallenbeck@dataspeedinc.com> - 2.1.10-1
- Autogenerated by Bloom

* Fri Feb 23 2024 Kevin Hallenbeck <khallenbeck@dataspeedinc.com> - 2.1.9-1
- Autogenerated by Bloom

* Tue Feb 20 2024 Kevin Hallenbeck <khallenbeck@dataspeedinc.com> - 2.1.8-1
- Autogenerated by Bloom

* Thu Sep 14 2023 Kevin Hallenbeck <khallenbeck@dataspeedinc.com> - 2.1.3-1
- Autogenerated by Bloom

* Wed Jan 25 2023 Kevin Hallenbeck <khallenbeck@dataspeedinc.com> - 2.1.1-1
- Autogenerated by Bloom

* Fri Dec 02 2022 Kevin Hallenbeck <khallenbeck@dataspeedinc.com> - 2.1.0-1
- Autogenerated by Bloom

