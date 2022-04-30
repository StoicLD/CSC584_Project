using UnityEngine.UI;

namespace AI
{
    public class ActionNode : DecisionTreeNode
    {
        private bool isActive = false;

        public bool IsActive
        {
            get => isActive;
            set => isActive = value;
        }

        public override DecisionTreeNode MakeDecision()
        {
            return this;
        }

        public virtual void UpdateAction()
        {
            if (!isActive) return;
            //do action
        }

    }
}