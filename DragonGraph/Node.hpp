#pragma once

#include "Namespace.hpp"
#include <type_traits>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>
#include "NoCopy.hpp"

GNMGRAPH_NAMESPACE_OPEN

/////////////////// z1 ATTRIBUTE TYPES

struct AttrType {
  enum E {
    kInt = 0,
    kDouble = 1,
    kFloat = 2

  };

  enum ET {
    kFIndexMeshData = 3,        // index mesh data w float pts
    kIndexMeshData = 4,         // index mesh data w double pts
    kIndexMeshConnectivity = 5, // index mesh data without points

  };
};

/////////////////// z2 SPECIAL TYPES

struct IndexMeshData_impl {
  int *indices;
  int *counts;
  double *points;

  int indices_sz;
  int counts_sz;
  int points_sz;

  IndexMeshData_impl(int *in_indices, int *in_counts, double *in_points,

                     int in_indices_sz, int in_counts_sz, int in_points_sz)
      :

        indices(in_indices), counts(in_counts), points(in_points)

        ,
        indices_sz(in_indices_sz), counts_sz(in_counts_sz),
        points_sz(in_points_sz)

  {}
};

struct FIndexMeshData_impl {
  int *indices;
  int *counts;
  float *points;

  int indices_sz;
  int counts_sz;
  int points_sz;

  FIndexMeshData_impl(int *in_indices, int *in_counts, float *in_points,

                      int in_indices_sz, int in_counts_sz, int in_points_sz)
      :

        indices(in_indices), counts(in_counts), points(in_points)

        ,
        indices_sz(in_indices_sz), counts_sz(in_counts_sz),
        points_sz(in_points_sz)

  {}
};

namespace {

template <typename... Indices>
IndexMeshData_impl *ct_tri_mesh_topology(Indices... indices) {
  constexpr int sz = sizeof...(indices);

  auto indices_arr = new int[sz]{indices...};
  auto counts = new int[sz / 3];
  memset(counts, 3, sizeof(int) * (sz / 3));

  return new IndexMeshData_impl(

      indices_arr, counts, (double *)0,

      sz, (sz / 3), 0);
#if 0
#endif

  return nullptr;
}

} // namespace

static void attribute_assign(IndexMeshData_impl &lval,
                             IndexMeshData_impl &rval) {

  memcpy(lval.indices, rval.indices, sizeof(int) * rval.indices_sz);
  memcpy(lval.counts, rval.counts, sizeof(int) * rval.counts_sz);
  memcpy(lval.points, rval.points, sizeof(double) * rval.points_sz);
}

static void attribute_assign(FIndexMeshData_impl &lval,
                             FIndexMeshData_impl &rval) {

  memcpy(lval.indices, rval.indices, sizeof(int) * rval.indices_sz);
  memcpy(lval.counts, rval.counts, sizeof(int) * rval.counts_sz);
  memcpy(lval.points, rval.points, sizeof(float) * rval.points_sz);
}

//////////////////// z7

template <int> struct Ct_ATypeMap;

template <> struct Ct_ATypeMap<AttrType::kInt> { using type = int; };

template <> struct Ct_ATypeMap<AttrType::kDouble> { using type = double; };
template <> struct Ct_ATypeMap<AttrType::kFloat> { using type = float; };

template <> struct Ct_ATypeMap<AttrType::kIndexMeshData> {
  using type = IndexMeshData_impl;
};
template <> struct Ct_ATypeMap<AttrType::kFIndexMeshData> {
  using type = FIndexMeshData_impl;
};

/////////////////// z3 ATTRIBUTES

struct AttrValue {
  union {
    double numeric;
    void *ptr;
  };
  AttrValue() {}
  AttrValue(double val) { numeric = val; }
  AttrValue(void *val) { ptr = val; }
};

class Attribute {
public:
  int type;
  AttrValue actual_value; // only for numerics
  double default_value;   // only for numerics
  double min_value;       // only for numerics
  double max_value;       // only for numerics
  Attribute() { memset(this, 0, sizeof(Attribute)); }

  Attribute(int intype) {
    memset(this, 0, sizeof(Attribute));
    type = intype;
  }

  Attribute(int intype, double indefault)
      : type(intype), actual_value(indefault), default_value(indefault) {

    min_value = std::numeric_limits<double>::min();
    max_value = std::numeric_limits<double>::max();
  }

  Attribute(int intype, double indefault, double min)
      : type(intype), actual_value(indefault), default_value(indefault),
        min_value(min) {

    max_value = std::numeric_limits<double>::max();
  }

  Attribute(int intype, double indefault, double min, double max)
      : type(intype), actual_value(indefault), default_value(indefault),
        min_value(min), max_value(max) {}
};

class NamedAttr : public Attribute {
public:
  std::string name;
  NamedAttr() { name = "noname"; }

  template <class... Args>
  NamedAttr(std::string const &inname, Args... args)
      : Attribute(args...), name(inname) {}
};

/////////////////// z4 LINK

// TODO : implement move copy ctor
struct AttrRelations {

  std::vector<NamedAttr> inputs;
  std::vector<NamedAttr> outputs;

  std::vector<int> input_to_outputs; // indices to outputs vector 
  std::vector<int> offset; // offset of input i in input_to_outputs 
};

// class only used once , at the stage where we define the attrs of a node
class AttrManager {

  std::vector<NamedAttr> inputs;
  std::vector<NamedAttr> outputs;

  std::vector<int> input_to_outputs; // pointers of outputs

#ifndef NDEBUG
  std::vector<std::string> input_to_outputs_str; // pointers of outputs

#endif

  std::vector<int> offset;

  int last_linked_input = 0;
  int last_offset = 0;

public:
  AttrManager() {}

  template <class... Ins> void addInputs(Ins... ins) {
    assert(inputs.empty() && outputs.empty() && "add inputs firts!");
    int a[] = {0, (inputs.push_back(ins), 0)...};
    static_cast<void>(a); // unused
  }
  template <class... Outs> void addOutputs(Outs... outs) {
    int a[] = {0, (outputs.push_back(outs), 0)...};
    static_cast<void>(a); // unused
  }

private:
  static int find_str(const NamedAttr *attrs, int sz, const char *name) {
    for (size_t i = 0; i < sz; i++) {
      if (attrs[i].name == name) {
        return i;
      }
    }
    return -1;
  }

public:
  // input attributes MUST be linked in order
  // this means an input MUST have an affect
  template <class... ConstChars>
  void link(const char *input, ConstChars... outs) {
    const char *a[] = {outs...};
    constexpr size_t num_outs = sizeof...(outs);
    static_assert(num_outs >= 1, "string->string(,string,...)");

    if (offset.empty()) {
      offset.resize(inputs.size());
    }

    for (size_t i = 0; i < num_outs; i++) {

      int idx = find_str(outputs.data(), outputs.size(), a[i]);
      assert(idx != -1 && "name not found on any attribute");
      input_to_outputs.push_back(idx);

#ifndef NDEBUG
      input_to_outputs_str.push_back(a[i]);
#endif
    }
    offset[last_linked_input] = last_offset;
    last_offset += num_outs;
    last_linked_input += 1;
  }

#ifndef NDEBUG
  void recap_link() {

    for (size_t i = 0; i < inputs.size() - 1; i++) {
      std::cout << "input " << i << '\n';
      std::cout << "is connected to :\n";

      int len = offset[i + 1];
      for (size_t j = offset[i]; j < len; j++) {
        std::cout << input_to_outputs_str[j] << ",";
      }
      std::cout << '\n';
    }

    int i = inputs.size() - 1;
    std::cout << "input " << i << '\n';
    std::cout << "is connected to :\n";

    for (size_t j = offset[i]; j < input_to_outputs_str.size(); j++) {
      std::cout << input_to_outputs_str[j] << ",";
    }

    std::cout << '\n';
  }

#endif

  void move_self_into_rel(AttrRelations &relation) {

    std::vector<Attribute> new_outputs;
    std::vector<Attribute *> new_relmap;
    new_outputs.reserve(outputs.size());
    new_relmap.reserve(input_to_outputs.size());

    // for (auto &cur : outputs) {
    //  new_outputs.push_back(cur);
    //}

    relation.inputs = std::move(inputs);
    relation.outputs = std::move(outputs);
    relation.input_to_outputs = std::move(input_to_outputs);
    relation.offset = std::move(offset);
  }
};

// after using the attrmanager we will compress its data

/////////////////// z5 NODE

class NodeBase {

public:
  NodeBase() {}

  virtual void compute(std::vector<std::string> names) {}

  // to set the dep between output and input
  static AttrRelations init() {

    /*
       AttrManager mana;

  mana.addInputs(NamedAttr{"bella"}, NamedAttr{"porto"});
  mana.addOutputs(NamedAttr{"shia"}, NamedAttr{"donna"});
  mana.link("bella", "shia", "donna");
  mana.link("porto", "donna");
  mana.recap_link();


  AttrRelations rel;


  mana.move_self_into_rel(rel);
    */
  }

  static void *creator() { return nullptr; }
};

struct GraphConnec {

  int src_node;
  std::vector<NamedAttr *> src_handles;

  int dst_node;
  std::vector<NamedAttr *> dst_handles;
};
size_t encode_orderer_intpair(int a, int b) {

  size_t c = a < b ? a : b;
  c = c << 32;
  c |= (a > b ? a : b);
  ;
  return c;
}

#if 0
class GraphConnectionHeap {
 

    std::vector<int> handles;
    std::vector<int> offset; 
 

 public:
   GraphConnectionHeap() { 
   
   } 

} ;

#endif

class Graph {
public:
  int head_node;

  int eval_;

  std::vector<NodeBase *> node;
  std::vector<AttrRelations> node_io; // 1 : 1 with node

  std::vector<GraphConnec> connect;

  Graph() { eval_ = 0; }

  GNM_NOCOPY(Graph);

  void set_head_node(int hdl) { head_node = hdl; }

  void touch(const char *attr_name) {

    if (eval_) { 
         auto &inp = node_io[head_node].inputs;
         auto &out = node_io[head_node].outputs;
         auto &offset = node_io[head_node].offset;
         auto &mapper = node_io[head_node].offset; 
         int aa = find_attr_byname(inp.data(), inp.size(), attr_name);

          auto zz = offset[aa]; 
          auto yy = mapper[zz];
            auto xx =  out [yy];

             std::vector<std::string> vec; 
              node[head_node]->compute( vec ); 
              
      // std::vector<std::string> vec;
      // vec.push_back(->name);
      // node[head_node]->compute(vec);
    }
  };

  void recompute_all() {
    
  }

  static int find_attr_byname(const NamedAttr *node_affect, const int sz,
               const char * attr_name  )  {
    for (int i = 0  ; i < sz; ++i) {
      if (node_affect[i].name == attr_name) {
        return i; 
         }   
    }
    return -1 ;
  }

  void connect_attr(int src_hdl, const char *src_plug, int dst_hdl,
                    const char *dst_plug) {
    connect.resize(node.size());

    auto &out = node_io[src_hdl].outputs;
    int src_i = find_attr_byname(out.data(), out.size(), src_plug);

    auto &inp = node_io[dst_hdl].inputs;
    int dst_i = find_attr_byname(inp.data(), inp.size(), dst_plug);

    connect[src_hdl].dst_node = dst_hdl;
    connect[src_hdl].dst_handles.push_back(&inp[dst_i]);

    connect[dst_hdl].src_node = src_hdl;
    connect[dst_hdl].src_handles.push_back(&out[src_i]);
  }

  static bool type_is_special(int type) {

    if (type == AttrType::kFloat) {
      return false;
    } else if (type == AttrType::kInt) {
      return false;

    } else if (type == AttrType::kDouble) {
      return false;
    }

    return true;
  }

  static void cast_and_assign(AttrValue &lval, void *rval, int type) {

    if (type == AttrType::kInt) {
      lval.numeric = *(int *)rval;
    } else if (type == AttrType::kDouble) {
      lval.numeric = *(double *)rval;
    } else if (type == AttrType::kIndexMeshData) {
      attribute_assign(*(IndexMeshData_impl *)lval.ptr,
                       *(IndexMeshData_impl *)rval);
    }
  }

  template <class T>
  T   get_numerical_ct(int node_hdl, const char *attr_name) {


    auto &inp = node_io[node_hdl].inputs;
    int idx = find_attr_byname(inp.data(), inp.size(), attr_name);
    assert(idx != -1);
   
    return (T  )inp[idx].actual_value.numeric;
  }

  template <class T>
  T const &get_special_ct(int node_hdl, const char *attr_name) {

    auto &inp = node_io[node_hdl].inputs;
    int i = -1;
    (void)std::find_if(inp.begin(), inp.end(), [&i, attr_name](auto arg) {
      ++i;
      return arg.name == attr_name;
    });
    assert(i != inp.size());
    return *(T *)inp[i].actual_value.ptr;
  }

  void *get_value_ptr(int node_hdl, const char *attr_name) {
    auto &inp = node_io[node_hdl].inputs;
    int i = find_attr_byname(inp.data(), inp.size(), attr_name);
    assert(i >= 0);
    assert(i != inp.size());
    assert(type_is_special(inp[i].type));
    return inp[i].actual_value.ptr;
  }

  void set_value_ptr(int node_hdl, const char *attr_name, void *data) {

    auto &inp = node_io[node_hdl].inputs;
    int idx = find_attr_byname(inp.data(), inp.size(), attr_name);
    assert(idx != inp.size());
#ifndef NDEBUG
    type_is_special(inp[idx].type);
#endif
    inp[idx].actual_value.ptr = data;
  }

  template <class T>
  void set_value(int node_hdl, const char *attr_name, const T &data) {

    auto &inp = node_io[node_hdl].inputs;
    int idx = find_attr_byname(inp.data(), inp.size(), attr_name);
    assert(idx != -1);
   

    cast_and_assign(inp[idx].actual_value, (void *)&data, inp[idx].type);
  
  }

  int spawn_node(void *(*creator_func)(void),
                 AttrRelations (*init_func)(void)) {

    AttrRelations rel = init_func();
    node_io.push_back(rel);

    NodeBase *new_node = (NodeBase *)creator_func();
    assert(new_node != nullptr);
    node.push_back(new_node);
    int hdl = node.size() - 1;
    return hdl;
  }

  ~Graph() {

    for (auto &ptr : node) {
      ptr = nullptr;
      delete ptr;
    }
  }
};

GNMGRAPH_NAMESPACE_CLOSE
