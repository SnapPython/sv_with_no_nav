//still some problem with the depend


// Simple function that return a NodeStatus
// BT::NodeStatus ifOrdered()
// {
//   std::cout << "[ Battery: OK ]" << std::endl;
//   return BT::NodeStatus::SUCCESS;
// }

class Decision : public BT::SyncActionNode{

public:
  
  void init(){
    BehaviorTreeFactory factory;
    auto BT::tree = factory.createTreeFromText(../config/sentry_bt.xml);
    BT::Groot2Publisher publisher(tree);
  }

  BT::NodeStatus ifOrdered(){
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus goToPlace(){
    return BT::NodeStatus::SUCCESS;

  }

  BT::NodeStatus ifHighHp(){
    return BT::NodeStatus::SUCCESS;

  }

  BT::NodeStatus goAround(){
    return BT::NodeStatus::SUCCESS;

  }

  BT::NodeStatus ifAddHpOk(){
    return BT::NodeStatus::SUCCESS;

  }

  BT::NodeStatus goToBase(){
    return BT::NodeStatus::SUCCESS;

  }

};