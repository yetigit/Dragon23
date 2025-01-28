#include <DragonGraph/Node.hpp>

#include <iostream>

using namespace gnmgraph;

struct A : public NodeBase {

  static void *creator() { return new A(); }
  void compute( std::vector<std::string> names  ) override { 
         

      std::cout << "compute from A () \n"; 
    }  


  static AttrRelations init() {

    AttrManager mana;

    mana.addInputs(NamedAttr{"bella", AttrType::kInt, 1.0, 0.0, 3.0},
                   NamedAttr{"porto", AttrType::kDouble, 2.0, 0.5, 8.0});

    mana.addOutputs(NamedAttr{"shia", AttrType::kInt},
                    NamedAttr{"donna", AttrType::kInt});

    mana.link("bella", "shia", "donna");
    mana.link("porto", "donna");
    //   mana.recap_link();

    AttrRelations rel;

    mana.move_self_into_rel(rel);

    return rel;
  }
};

struct B : public NodeBase {

  B() {}

  static void *creator() { return new B(); } 

    void compute(std::vector<std::string> names) override {

    std::cout << "compute from B () \n";
  }  



  static AttrRelations init() {

    AttrManager mana;

    mana.addInputs(NamedAttr{"bella", AttrType::kInt, 1.0, 0.0, 3.0},
                   NamedAttr{"porto", AttrType::kDouble, 2.0, 0.5, 8.0});

    mana.addOutputs(NamedAttr{"shia", AttrType::kInt},
                    NamedAttr{"donna", AttrType::kInt});

    mana.link("bella", "shia", "donna");
    mana.link("porto", "donna");
    // mana.recap_link();

    AttrRelations rel;

    mana.move_self_into_rel(rel);

    return rel;
  }
};

struct C : public NodeBase {

  C() {}

    void compute(std::vector<std::string> names) override {

    std::cout << "compute from A () \n";
  }  


  static void *creator() { return new C(); }
  static AttrRelations init() {

    AttrManager mana;

    mana.addInputs(NamedAttr{"bella", AttrType::kIndexMeshData});
    mana.addOutputs(NamedAttr{"tomate", AttrType::kIndexMeshData});

    mana.link("bella", "tomate");

    AttrRelations rel;

    mana.move_self_into_rel(rel);

    return rel;
  }
};

int main() {

  // AttrRelations rel_a = A::init();
  // AttrRelations rel_b = B::init();

  // A a;
  // B b;

  Graph graph;

  int hdl_a = graph.spawn_node(A::creator, A::init);
  int hdl_b = graph.spawn_node(B::creator, B::init);

   graph.set_value(hdl_a, "bella",  21);
  std::cout << graph.get_numerical_ct<int>(hdl_a, "bella") << '\n';
  
  // graph.set_value(hdl_a, "porto", 3.14);
  // 
  // 
    graph.connect_attr(hdl_a, "shia",  hdl_b, "bella"  );
 

  graph.eval_ = 1;
  graph.head_node = hdl_a;    
  graph.touch("bella")  ; 
     
  return 0;
}
 