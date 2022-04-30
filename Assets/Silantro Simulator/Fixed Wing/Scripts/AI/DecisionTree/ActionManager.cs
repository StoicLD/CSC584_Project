using System;
using System.Collections.Generic;
using UnityEngine;

namespace AI
{
    public class ActionManager : MonoBehaviour
    {
        private static ActionManager instance;

        public static ActionManager Instance => instance;

        private List<ActionNode> actions = new List<ActionNode>();
        
        private void Awake()
        {
            if (!instance)
            {
                instance = this;
                DontDestroyOnLoad(gameObject);
            }
            else
            {
                Destroy(gameObject);
            }
        }

        public void AddAction(ActionNode act)
        {
            actions.Add(act);
        } 

        private void LateUpdate()
        {
            foreach (var action in actions)
            {
                action.UpdateAction();
            }
        }
    }
}