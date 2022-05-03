using System;
using System.Collections;
using UnityEngine;

namespace AI
{
    class act1 : ActionNode
    {
        private Transform obj;

        public Transform Obj
        {
            get => obj;
            set => obj = value;
        }

        public override void UpdateAction()
        {
            if (!IsActive) return;
            obj.Translate(new Vector3(1 * Time.deltaTime,0,0),Space.World);
        }
    }
    
    class act2 : ActionNode
    {
        private Transform obj;

        public Transform Obj
        {
            get => obj;
            set => obj = value;
        }

        public override void UpdateAction()
        {
            if (!IsActive) return;
            obj.Translate(new Vector3(0,1 * Time.deltaTime,0),Space.World);
        }
    }
    
    class act3 : ActionNode
    {
        private Transform obj;

        public Transform Obj
        {
            get => obj;
            set => obj = value;
        }

        public override void UpdateAction()
        {
            if (!IsActive) return;
            obj.Rotate(obj.up, 10);
        }
    }

    class dec1 : DecisionNode
    {
        private Transform obj;

        public Transform Obj
        {
            get => obj;
            set => obj = value;
        }
        
        public override DecisionTreeNode GetBranch()
        {
            if (obj.position.x > 5)
            {
                return TrueNode;
            }

            return FalseNode;
        }
    }
    
    class dec2 : DecisionNode
    {
        private bool isTrue = false;
        
        public bool IsTrue
        {
            get => isTrue;
            set => isTrue = value;
        }

        public override DecisionTreeNode GetBranch()
        {
            if (isTrue)
            {
                return TrueNode;
            }

            return FalseNode;
        }
    }
    public class TestCase : MonoBehaviour
    {
        public Transform obj;

        private dec1 dec1 = new dec1();
        private dec2 dec2 = new dec2();
        private act1 act1 = new act1();
        private act2 act2 = new act2();
        private act3 act3 = new act3();

        private DecisionTree tree = new DecisionTree();
        
        void Init()
        {
            if (obj)
            {
                dec1.Obj = obj;
                act1.Obj = obj;
                act2.Obj = obj;
                act3.Obj = obj;
                
                ActionManager.Instance.AddAction(act1);
                ActionManager.Instance.AddAction(act2);
                ActionManager.Instance.AddAction(act3);
            }
        }
        private void Start()
        {
            Init();
            tree.Root = dec2;
            dec2.FalseNode = act3;
            dec2.TrueNode = dec1;

            dec1.TrueNode = act2;
            dec1.FalseNode = act1;
        }

        private void Update()
        {
            if (Input.anyKeyDown)
            {
                dec2.IsTrue = true;
            }
            tree.UpdateTree();
        }
    }
}