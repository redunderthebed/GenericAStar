using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using generic_astar;
using System.Linq;

namespace GenericAStar.Test
{
    [TestClass]
    public class TestAStarNode
    {
        [TestMethod]
        public void shouldBeOrderableByFScore()
        {
            NavMap map = new NavMap();
            NavNode a = new NavNode(new Point(2, 3));
            NavNode b = new NavNode(new Point(3, 3));
            NavNode c = new NavNode(new Point(1, 1));
            map.Nodes.Add(a);
            map.Nodes.Add(b);
            map.Nodes.Add(c);
            AStarNode node = new AStarNode(a, b, 10, map);
            AStarNode other = new AStarNode(c, b, 10, map);

            List<AStarNode> set = new List<AStarNode>();
            node.GScore = 10;
            other.GScore = 10;
            set.Add(other);
            set.Add(node);

            set.Sort(new FScoreComparer());
            Assert.IsTrue(set[0].FScore < set[1].FScore);
            AStarNode first = set.First();
            Assert.AreEqual(first, set[0]);
            Assert.AreEqual(first, node);

            other.GScore = 0;
            set.Sort(new FScoreComparer());
            Assert.IsTrue(set[0].FScore < set[1].FScore);
            first = set.First();
            Assert.AreEqual(first, set[0]);
            Assert.AreEqual(first, other);

        }
    }
    [TestClass]
    public class TestGOAPNode
    {
        IEnumerable<WorldStateToken> tokens;
        IEnumerable<WorldStateToken> goalTokens;
        WorldState ws;

        [TestInitialize]
        public void Initialize()
        {
            tokens = new List<WorldStateToken> { new WorldStateToken("a", true), new WorldStateToken("b", false), new WorldStateToken("c", true) };
            goalTokens = new List<WorldStateToken> { new WorldStateToken("b", true), new WorldStateToken("c", false), new WorldStateToken("d", true) };
            ws = new WorldState();
            foreach (WorldStateToken token in tokens)
            {
                ws.SetToken(token);
            }
        }
        [TestMethod]
        public void ShouldBeOrderableByFScore()
        {
            


        }
        [TestMethod]
        public void ShouldHaveSameTokensAsClone()
        {
            
            foreach (WorldStateToken token in tokens)
            {
                ws.SetToken(token);
            }
            generic_astar.Action action = new generic_astar.Action("test_action", new List<WorldStateToken>() { }, new List<WorldStateToken>() { }, 10);
            GOAPNode node = new GOAPNode(ws, action);
            GOAPNode clone = (GOAPNode)node.Clone();
            foreach (WorldStateToken token in tokens)
            {
                Assert.AreEqual(token.Value, clone.WorldState.GetValue(token.Name));
            }
        }

        [TestMethod]
        public void ShouldBeIndependentOfClone()
        {
            GOAPNode original = new GOAPNode(new WorldState(ws), null);
            GOAPNode clone = (GOAPNode)original.Clone();

            clone.WorldState.SetToken("a", !(bool)original.WorldState.GetValue("a"));
            Assert.AreNotEqual(clone.WorldState.GetValue("a"), original.WorldState.GetValue("a"));
        }

    }
    [TestClass]
    public class TestWorldState
    {
        IEnumerable<WorldStateToken> tokens;
        IEnumerable<WorldStateToken> goalTokens;

        [TestInitialize]
        public void Initialize()
        {
            tokens = new List<WorldStateToken> { new WorldStateToken("a", true), new WorldStateToken("b", false), new WorldStateToken("c", true) };
            goalTokens = new List<WorldStateToken> { new WorldStateToken("b", true), new WorldStateToken("c", false), new WorldStateToken("d", true) };
        }

        [TestMethod]
        public void ShouldSetTokens()
        {
            WorldState state = new WorldState();
            foreach (WorldStateToken token in tokens)
            {
                state.SetToken(token);
            }
            foreach (WorldStateToken token in tokens)
            {
                Assert.AreEqual(token.Value, state.GetValue(token.Name));
            }
            foreach (WorldStateToken token in goalTokens)
            {
                state.SetToken(token);
            }
            foreach (WorldStateToken token in goalTokens)
            {
                Assert.AreEqual(token.Value, state.GetValue(token.Name));
            }
        }

        [TestMethod]
        public void ShouldBeSatisfiedBySame()
        {
            WorldState state = new WorldState();
            WorldState goal = new WorldState();
            foreach (WorldStateToken token in goalTokens)
            {
                state.SetToken(token.Name, token.Value);
                goal.SetToken(token.Name, token.Value);
            }
            Assert.IsTrue(goal.SatisfiedBy(state));
        }

        [TestMethod]
        public void ShouldBeSatisfiedBySuperset()
        {
            WorldState state = new WorldState();
            WorldState goal = new WorldState();
            foreach (WorldStateToken token in tokens)
            {
                state.SetToken(token.Name, token.Value);
            }
            foreach (WorldStateToken token in goalTokens)
            {
                state.SetToken(token.Name, token.Value);
                goal.SetToken(token.Name, token.Value);
            }
            Assert.IsTrue(goal.SatisfiedBy(state));
        }

        [TestMethod]
        public void ShouldNotBeSatisfiedBySubset()
        {
            WorldState state = new WorldState();
            WorldState goal = new WorldState();
            foreach (WorldStateToken token in tokens)
            {
                state.SetToken(token.Name, token.Value);
            }
            foreach (WorldStateToken token in goalTokens.Take(goalTokens.Count() - 1))
            {
                state.SetToken(token.Name, token.Value);

            }
            foreach (WorldStateToken token in goalTokens.Take(goalTokens.Count()))
            {
                goal.SetToken(token.Name, token.Value);
            }
            Assert.IsFalse(goal.SatisfiedBy(state));
        }

        [TestMethod]
        public void ShouldBeOneDifferentPerDifferentWorldToken()
        {
            WorldState state = new WorldState();
            WorldState different = new WorldState();
            foreach(WorldStateToken token in tokens)
            {
                state.SetToken(token);
                different.SetToken(token);
            }
            int count = 0;
            foreach(WorldStateToken token in different)
            {
                token.Value = !token.Value;
                count++;
                Assert.AreEqual(count, state.DifferenceFrom(different));
            }
        }

        [TestMethod]
        public void ShouldBeOneDifferentPerMissingInnerWorldToken()
        {
            WorldState state = new WorldState();
            WorldState different = new WorldState();
            foreach (WorldStateToken token in tokens)
            {
                state.SetToken(token);
            }
            int count = state.Count();
            foreach (WorldStateToken token in different)
            {
                state.SetToken(token.Name, token.Value);
                count--;
                Assert.AreEqual(count, state.DifferenceFrom(different));
            }
        }

        [TestMethod]
        public void ShouldBeOneDifferentPerMissingOuterWorldToken()
        {
            WorldState state = new WorldState();
            WorldState different = new WorldState();
            foreach (WorldStateToken token in tokens)
            {
                state.SetToken(token);

            }
            foreach (WorldStateToken token in state)
            {
                different.SetToken(token.Name, token.Value);
                Assert.AreEqual(0, state.DifferenceFrom(different));
            }
        }

        [TestMethod]
        public void ShouldNotBeEqualToSuperset()
        {
            WorldState state = new WorldState();
            WorldState different = new WorldState();
            foreach (WorldStateToken token in tokens.Take(2))
            {
                state.SetToken(token);
            }
            foreach (WorldStateToken token in tokens)
            {
                different.SetToken(token);
            }
            Assert.AreNotEqual(state, different);
        }

        [TestMethod]
        public void ShouldNotBeEqualToSubset()
        {
            WorldState state = new WorldState();
            WorldState different = new WorldState();
            foreach (WorldStateToken token in tokens)
            {
                state.SetToken(token);
            }
            foreach (WorldStateToken token in tokens.Take(2))
            {
                different.SetToken(token);
            }
            Assert.AreNotEqual(state, different);
        }
    }

}
