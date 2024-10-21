#!/usr/bin/env python3

# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import re
import sys
try:
    import importlib.resources as importlib_resources
except ModuleNotFoundError:
    import importlib_resources

IS_WINDOWS = os.name == 'nt'


def get_environment_hook_template_path(name):
    with importlib_resources.path('ament_package.template.environment_hook', name) as path:
        return str(path)


def get_package_level_template_names(all_platforms=False):
    names = ['local_setup.%s.in' % ext for ext in [
        'bash',
        'bat',
        'sh',
        'zsh',
    ]]
    if not all_platforms:
        names = [name for name in names if _is_platform_specific_extension(name)]
    return names


def get_package_level_template_path(name):
    with importlib_resources.path('ament_package.template.package_level', name) as path:
        return str(path)


def get_prefix_level_template_names(*, all_platforms=False):
    extensions = [
        'bash',
        'bat.in',
        'sh.in',
        'zsh',
    ]
    names = ['local_setup.%s' % ext for ext in extensions] + \
        ['setup.%s' % ext for ext in extensions] + \
        ['_local_setup_util.py']
    if not all_platforms:
        names = [name for name in names if _is_platform_specific_extension(name)]
    return names


def get_prefix_level_template_path(name):
    with importlib_resources.path('ament_package.template.prefix_level', name) as path:
        return str(path)


def get_isolated_prefix_level_template_names(*, all_platforms=False):
    extensions = [
        'bash',
        'bat.in',
        'sh.in',
        'zsh',
    ]
    names = ['local_setup.%s' % ext for ext in extensions] + \
        ['_order_isolated_packages.py']
    # + ['setup.%s' % ext for ext in extensions]
    if not all_platforms:
        names = [name for name in names if _is_platform_specific_extension(name)]
    return names


def get_isolated_prefix_level_template_path(name):
    with importlib_resources.path('ament_package.template.isolated_prefix_level', name) as path:
        return str(path)


def configure_file(template_file, environment):
    """
    Evaluate a .in template file used in CMake with configure_file.

    :param template_file: path to the template, ``str``
    :param environment: dictionary of placeholders to substitute,
      ``dict``
    :returns: string with evaluates template
    :raises: KeyError for placeholders in the template which are not
      in the environment
    """
    with open(template_file, 'r') as f:
        template = f.read()
        return configure_string(template, environment)


def configure_string(template, environment):
    """
    Substitute variables enclosed by @ characters.

    :param template: the template, ``str``
    :param environment: dictionary of placeholders to substitute,
      ``dict``
    :returns: string with evaluates template
    :raises: KeyError for placeholders in the template which are not
      in the environment
    """
    def substitute(match):
        var = match.group(0)[1:-1]
        if var in environment:
            return environment[var]
        return ''
    return re.sub(r'\@[a-zA-Z0-9_]+\@', substitute, template)


def _is_platform_specific_extension(filename):
    if filename.endswith('.in'):
        filename = filename[:-3]
    if not IS_WINDOWS and filename.endswith('.bat'):
        # On non-Windows system, ignore .bat
        return False
    if IS_WINDOWS and os.path.splitext(filename)[1] not in ['.bat', '.py']:
        # On Windows, ignore anything other than .bat and .py
        return False
    return True

IS_WINDOWS = os.name == 'nt'


def main(argv=sys.argv[1:]):
    """
    Extract the information about templates provided by ament_package.

    Call the API provided by ament_package and
    print CMake code defining several variables containing information about
    the available templates.
    """
    parser = argparse.ArgumentParser(
        description='Extract information about templates provided by '
                    'ament_package and print CMake code defining several '
                    'variables',
    )
    parser.add_argument(
        'outfile',
        nargs='?',
        help='The filename where the output should be written to',
    )
    args = parser.parse_args(argv)

    lines = generate_cmake_code()
    if args.outfile:
        basepath = os.path.dirname(args.outfile)
        if not os.path.exists(basepath):
            os.makedirs(basepath)
        with open(args.outfile, 'w') as f:
            for line in lines:
                f.write('%s\n' % line)
    else:
        for line in lines:
            print(line)


def generate_cmake_code():
    """
    Return a list of CMake set() commands containing the template information.

    :returns: list of str
    """
    variables = []

    if not IS_WINDOWS:
        variables.append((
            'ENVIRONMENT_HOOK_LIBRARY_PATH',
            '"%s"' % get_environment_hook_template_path('library_path.sh')))
    else:
        variables.append(('ENVIRONMENT_HOOK_LIBRARY_PATH', ''))

    ext = '.bat.in' if IS_WINDOWS else '.sh.in'
    variables.append((
        'ENVIRONMENT_HOOK_PYTHONPATH',
        '"%s"' % get_environment_hook_template_path('pythonpath' + ext)))

    templates = []
    for name in get_package_level_template_names():
        templates.append('"%s"' % get_package_level_template_path(name))
    variables.append((
        'PACKAGE_LEVEL',
        templates))

    templates = []
    for name in get_prefix_level_template_names():
        templates.append('"%s"' % get_prefix_level_template_path(name))
    variables.append((
        'PREFIX_LEVEL',
        templates))

    lines = []
    for (k, v) in variables:
        if isinstance(v, list):
            lines.append('set(ament_cmake_package_templates_%s "")' % k)
            for vv in v:
                lines.append('list(APPEND ament_cmake_package_templates_%s %s)'
                             % (k, vv))
        else:
            lines.append('set(ament_cmake_package_templates_%s %s)' % (k, v))
    # Ensure backslashes are replaced with forward slashes because CMake cannot
    # parse files with backslashes in it.
    return [line.replace('\\', '/') for line in lines]


if __name__ == '__main__':
    main()
