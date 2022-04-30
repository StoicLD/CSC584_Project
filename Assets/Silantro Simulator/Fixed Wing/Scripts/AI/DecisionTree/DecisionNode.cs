

namespace AI
{
    public class DecisionNode : DecisionTreeNode
    {
        private DecisionTreeNode trueNode;
        private DecisionTreeNode falseNode;

        public DecisionTreeNode TrueNode
        {
            get => trueNode;
            set => trueNode = value;
        }

        public DecisionTreeNode FalseNode
        {
            get => falseNode;
            set => falseNode = value;
        }

        public DecisionNode()
        {
            trueNode = null;
            falseNode = null;
        }

        public DecisionNode(DecisionTreeNode tn, DecisionTreeNode fn)
        {
            trueNode = tn;
            falseNode = fn;
        }
        
        public void SetNode(DecisionTreeNode tn, DecisionTreeNode fn)
        {
            trueNode = tn;
            falseNode = fn;
        }
        
        public virtual DecisionTreeNode GetBranch()
        {
            return null;
        }
        public override DecisionTreeNode MakeDecision()
        {
            return GetBranch().MakeDecision();
        }
    }
}