#pragma once

// Reads newline-terminated text commands from USB serial and runs them.
// Every command is answered by exactly one line: "OK <NAME>" or "ERR <NAME> <reason>".
// Query commands print their data lines before the OK.
namespace commands {

void poll();  // call every loop

}  // namespace commands
