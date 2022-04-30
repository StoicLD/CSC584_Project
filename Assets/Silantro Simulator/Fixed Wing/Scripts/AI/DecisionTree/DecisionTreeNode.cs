using System.Collections.Generic;
using UnityEngine;

namespace AI
{
    public abstract class DecisionTreeNode
    {
        public abstract DecisionTreeNode MakeDecision();
    }
}