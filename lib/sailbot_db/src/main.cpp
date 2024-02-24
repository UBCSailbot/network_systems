#include <boost/program_options.hpp>

#include "util_db.h"

namespace po = boost::program_options;

enum class DBCommand { Clear, Populate, DumpSensors, DumpGlobalPath };

std::string to_string(DBCommand c)
{
    switch (c) {
        case DBCommand::Clear:
            return "clear";
        case DBCommand::Populate:
            return "populate";
        case DBCommand::DumpSensors:
            return "dump-sensors";
        case DBCommand::DumpGlobalPath:
            return "dump-global-path";
    }
};

const std::map<DBCommand, std::string> DBCommandDesc{
  {DBCommand::Clear, "Clear the contents of a database collection"},
  {DBCommand::Populate, "Populate a database collection with random data"},
  {DBCommand::DumpSensors, "Dump all sensor data in the database collection"},
  {DBCommand::DumpGlobalPath, "Dump the Global Paths stored in the database collection"},
};

int main(int argc, char ** argv)
{
    // Formatting is weird, see: https://www.boost.org/doc/libs/1_63_0/doc/html/program_options/tutorial.html
    po::options_description o_desc("Options");
    o_desc.add_options()("help,h", "Help message")(
      "db-name", po::value<std::vector<std::string>>(), "Name of db collection to target")(
      to_string(DBCommand::Clear).c_str(), DBCommandDesc.at(DBCommand::Clear).c_str()),
      (to_string(DBCommand::Populate).c_str(), DBCommandDesc.at(DBCommand::Populate).c_str());

    po::positional_options_description po_desc;
    po_desc.add("db-name", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(o_desc).positional(po_desc).run(), vm);
    po::notify(vm);

    const std::string usage_instructions = [&o_desc]() {
        std::stringstream ss;
        ss << "Usage: sailbot_db DB-NAME [COMMAND]\n\n"
           // Need to separately print that DB-NAME is a positional argument
           << "DB-NAME: Name of db collection to target\n\n"
           << "COMMAND:\n"
           << o_desc << std::endl;
        return ss.str();
    }();

    if (vm.count("help") != 0U) {
        std::cout << usage_instructions << std::endl;
        return 0;
    }

    if (vm.count("db-name") != 0U) {
        std::cout << "TODO" << std::endl;
    } else {
        std::cout << usage_instructions << std::endl;
        return -1;
    }
}
