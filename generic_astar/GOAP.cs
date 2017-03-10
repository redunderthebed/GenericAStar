using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace generic_astar
{
    public class GOAPMap : AStarMap
    {
        //TODO: This should be a dictionary of worldstatetokens
        public Dictionary<string, Dictionary<bool, List<Action>>> effectActions;

        public GOAPMap()
        {
            this.effectActions = new Dictionary<string, Dictionary<bool, List<Action>>>();
        }

        /**
            Adds a new action to the set of available actions to build new AStarNodes from
        */
            
        public void AddAction(Action action)
        {
            foreach (WorldStateToken token in action.Effect)
            {
                if (!effectActions.ContainsKey(token.Name))
                {
                    this.effectActions[token.Name] = new Dictionary<bool, List<Action>>();
                }
                if (!effectActions[token.Name].ContainsKey(token.Value))
                {
                    effectActions[token.Name][token.Value] = new List<Action>();
                }
                effectActions[token.Name][token.Value].Add(action);
                //effectActions[token.Name][!token.Value].Remove(action);
            }
        }

        public override float GetHeuristic(AStarNode node)
        {
            GOAPNode state = ((GOAPNode)node.Value);
            return state.WorldState.DifferenceFrom(((GOAPNode)(node.Goal)).WorldState);
        }

        
        public override IEnumerable<AStarNode> GetNeighbours(AStarNode node)
        {
            //Find unsatisfied goal tokens
            GOAPNode goal = ((GOAPNode)(node.Goal));
            GOAPNode current = ((GOAPNode)(node.Value));
            IEnumerable<WorldStateToken> unsatisfied = goal.WorldState.Where( x =>
            {
                object value = current.WorldState.GetValue(x.Name);
                if(value != null)
                {
                    return ((bool)value) != x.Value;
                }
                else
                {
                    return true;
                }
            });

            //Find actions to satisfy them
            foreach (WorldStateToken token in unsatisfied)
            {
                IEnumerable<Action> actions = effectActions[token.Name][token.Value];
                foreach (Action action in actions)
                {
                    //Modify current and goal states to include the action
                    GOAPNode newState = (GOAPNode)current.Clone();
                    GOAPNode goalState = (GOAPNode)node.Goal.Clone();
                    newState.LastAction = action;
                    foreach (WorldStateToken effect in action.Effect)
                    {
                        newState.WorldState.SetToken(effect);
                    }
                    foreach (WorldStateToken precondition in action.Precondition)
                    {
                        goalState.WorldState.SetToken(precondition);
                    }
                    //Next
                    yield return new AStarNode(newState, goalState, action.Cost, this);

                }
            }
        }
    }

    public class GOAPNode : IAStarState, ICloneable
    {
        public WorldState WorldState;
        public Action LastAction;

        public GOAPNode(WorldState state, Action lastAction) 
        {
            this.WorldState = state;
            this.LastAction = lastAction;
        }

        public bool EqualState(IAStarState other)
        {
            if (other is GOAPNode)
            {
                return ((GOAPNode)other).WorldState.SatisiedBy(WorldState);
            }
            else
            {
                return false;
            }
        }

        public object Clone()
        {
            return new GOAPNode(new WorldState(this.WorldState), LastAction);
        }
    }

    public class WorldState : IEnumerable<WorldStateToken>
    {
        private Dictionary<string, WorldStateToken> Tokens;

        public WorldState(WorldState other)
        {
            this.Tokens = new Dictionary<string, WorldStateToken>();
            foreach(KeyValuePair<string, WorldStateToken> pair in other.Tokens)
            {
                this.Tokens.Add(pair.Key, pair.Value);
            }
            
        }

        public WorldState(Dictionary<string, WorldStateToken> tokens)
        {
            this.Tokens = tokens;
        }
        public WorldState()
        {
            this.Tokens = new Dictionary<string, WorldStateToken>();
        }

        public void SetToken(String name, bool value)
        {
            if (Tokens.ContainsKey(name))
            {
                Tokens[name].Value = value;
            }
            else
            {
                Tokens.Add(name, new WorldStateToken(name, value));
            }
        }
        public void SetToken(WorldStateToken token)
        {
            SetToken(token.Name, token.Value);
        }

        public IEnumerator<WorldStateToken> GetEnumerator()
        {
            return Tokens.Select(x => x.Value).GetEnumerator();
        }

        public bool SatisiedBy(WorldState other)
        {
            try
            {
                return this.All(x => other.Tokens[x.Name].Value == x.Value);
            }
            catch(KeyNotFoundException keyEx)
            {
                return false;
            }
        }

        public object GetValue(string name)
        {
            if (Tokens.ContainsKey(name))
            {
                return Tokens[name].Value;
            }
            else
            {
                return null;
            }
        }

        /**
            Returns the number of WorldStateTokens that must be changed to convert this WorldState into the
            provided parameter WorldState
            @param other The world state to compare to
        */
        public float DifferenceFrom(WorldState other)
        {
            float dif = 0;
            foreach (WorldStateToken token in other)
            {
                object value = this.GetValue(token.Name);
                if (value == null || (bool)value != token.Value)
                {
                    //NOTE: This seems like a small amount when compared to action costs
                    dif += 1;
                }
            }
            int extras = Math.Max(0, other.Count() - this.Count());
            return dif + extras;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public override string ToString()
        {
            return "{ " + String.Join(", ", Tokens.Select(x => x.ToString())) + "}";   
        }
    }

    public class Action
    {
        public WorldState Precondition;
        public WorldState Effect;
        public float Cost;
        public string Name;

        public Action(string name, List<WorldStateToken> pres, List<WorldStateToken> posts, float cost)
        {
            this.Precondition = new WorldState();
            foreach (WorldStateToken token in pres)
            {
                this.Precondition.SetToken(token);
            }
            this.Effect = new WorldState();
            foreach (WorldStateToken token in posts)
            {
                this.Effect.SetToken(token);
            }
            this.Name = name;
            this.Cost = cost;
        }
    }

    public class WorldStateToken
    {
        public string Name { get; set; }
        public bool Value { get; set; }

        public WorldStateToken(String name, bool value)
        {
            this.Name = name;
            this.Value = value;
        }

        public override string ToString()
        {
            return this.Name + ": " + this.Value;
        }
    }
}
