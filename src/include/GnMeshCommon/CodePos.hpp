#pragma once


// #include <GnMeshCommon/Namespace.hpp>
#include <string>


/// function to get debug info at a line where something might have happened 

namespace  
{ 
    // use the macro defined down here 
    std::string getCodePosString_(const char * _file, const char * _function, int _line)
    {
            std::string trace ; 
            trace += _file ;
            trace += ":"; 
            trace += _function ;
            trace += ":"; 
            trace += std::to_string(_line);
            return trace;
    }

} // namespace


#define GNM_CODEPOS_STR (::getCodePosString_(__FILE__, __FUNCTION__ , __LINE__))
