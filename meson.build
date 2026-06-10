project(
  'cquik',
  'c',
  version: '0.1.0',
  license: 'MIT',
  default_options: [
    'c_std=c99',
    'warning_level=3',
  ],
)

cc = meson.get_compiler('c')
cquik_inc = include_directories('.')
cquik_dep = declare_dependency(include_directories: cquik_inc)
munit_inc = include_directories('external/munit')

install_headers('cquik.h')

example_deps = [cquik_dep]
math_dep = cc.find_library('m', required: false)
if math_dep.found()
  example_deps += math_dep
endif

cquik_kuka_example = executable(
  'cquik-kuka-kr6-ik',
  'examples/kuka_kr6_ik.c',
  dependencies: example_deps,
  install: false,
)

test('example-kuka-kr6-ik', cquik_kuka_example)

munit_lib = static_library(
  'munit',
  'external/munit/munit.c',
  include_directories: munit_inc,
  override_options: [
    'c_std=c11',
    'warning_level=0',
  ],
  install: false,
)

munit_dep = declare_dependency(
  include_directories: munit_inc,
  link_with: munit_lib,
)

unit_deps = [cquik_dep, munit_dep]
if math_dep.found()
  unit_deps += math_dep
endif

cquik_unit_tests = executable(
  'cquik-tests',
  'tests/cquik_tests.c',
  dependencies: unit_deps,
  install: false,
)

test('unit', cquik_unit_tests)

cquik_include_twice_test = executable(
  'cquik-include-twice-test',
  'tests/include_twice.c',
  dependencies: example_deps,
  install: false,
)

test('include-twice', cquik_include_twice_test)
