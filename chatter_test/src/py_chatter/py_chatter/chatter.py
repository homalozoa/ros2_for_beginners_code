# Copyright 2022 Homalozoa
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

class Chatter():

    def __init__(self, str):
        self.name = str

    def get_chatter_name(self):
        return self.name


def main(args=None):
    py_chatter = Chatter("Bye ROS 2.")
    print(py_chatter.get_chatter_name())


if __name__ == '__main__':
    main()
