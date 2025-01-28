#pragma once

struct ItemFlags {

  enum f {

    DELETED = 1 << 0,
    TAGGED = 1 << 1,
    TAGGED_X = 1 << 2,
    COLLAPSIBLE = 1 << 3,

  };

  static bool is_on(const unsigned &  a, ItemFlags::f farg) { return a & farg; }
  static bool is_off(const unsigned &a, ItemFlags::f farg) {
    return !is_on(a, farg);
  }

  static void enable(unsigned & a, ItemFlags::f farg) { 
      a |= farg; 
  }

  static unsigned &  disable(unsigned & a, ItemFlags::f farg) { 
      return  (a &= ~farg ); 
  }

  static unsigned & merge_lhs(unsigned & a, unsigned const & b) { 
       return ( a |= b );  
  }


};
