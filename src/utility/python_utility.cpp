/*
 MIT License

 Copyright (c) 2017 SAE Institute Switzerland AG

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#include <utility/python_utility.h>
#include <sstream>
#include <engine/log.h>
#include <cctype>
#include <iostream>

py::object import(const std::string& module, const std::string& path, py::object& globals)
{
	{
		std::ostringstream oss;
		oss << "Loading python module: "<< module <<" with globals: "<< globals.str().cast<std::string>();
		//sfge::Log::GetInstance()->Msg(oss.str());
	}

    py::dict locals;
    locals["module_name"] = py::cast(module); // have to cast the std::string first
    locals["path"]        = py::cast(path);

    py::eval<py::eval_statements>(            // tell eval we're passing multiple statements
        "import imp\n"
        "new_module = imp.load_module(module_name, open(path), path, ('py', 'U', imp.PY_SOURCE))\n",
        globals,
        locals);
    return locals["new_module"];
}

std::string module2component(std::string& module_name)
{
	std::istringstream iss(module_name);
	std::string token;
	std::string class_name;
	while (std::getline(iss, token, '_'))
	{
		if(token.size() > 0)
		{
			char first = token.at(0);
			class_name += std::toupper(first);
			class_name += token.substr(1, token.size());
		}
	}
	return class_name;
}
