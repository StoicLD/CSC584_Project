using UnityEngine;

namespace AI
{
    public class DecisionTree
    {
        private DecisionTreeNode root = null;
        private ActionNode oldAction = null;
        private ActionNode newAction = null;

        public DecisionTreeNode Root
        {
            get => root;
            set => root = value;
        }

        public ActionNode OldAction
        {
            get => oldAction;
            set => oldAction = value;
        }

        public ActionNode NewAction
        {
            get => newAction;
            set => newAction = value;
        }

        public DecisionTreeNode MakeDecision()
        {
            return root.MakeDecision();
        }

        public void UpdateTree()
        {
            if (newAction != null)
            {
                newAction.IsActive = false;
            }
            oldAction = newAction;
            newAction = MakeDecision() as ActionNode ?? oldAction;
            newAction.IsActive = true;
        }
    }
}