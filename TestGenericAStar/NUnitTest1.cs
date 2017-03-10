using System;
using System.Collections.Generic;
using generic_astar;
using NUnit.Framework;

namespace TestGenericAStar
{
    //[TestFixture]
    public class TestWorldState
    {
        public void ShouldSetTokens()
        {
            WorldState state = new WorldState();
            WorldStateToken[] tokens = new WorldStateToken[] { new WorldStateToken("a", true), new WorldStateToken("b", false), new WorldStateToken("c", true) };
            foreach(WorldStateToken token in tokens)
            {
                state.SetToken(token);
            }
            foreach (WorldStateToken token in tokens)
            {
                Assert.AreEqual(token.Value, state.GetValue(token.Name));
            }

        }

        public void ShouldBeSatisfedBySame()
        {

        }

        public void ShouldNotBeSatisfiedBySuperset()
        {

        }

        public void ShouldNotBeSatisfiedBySubset()
        {

        }
    }
    //[TestFixture]
    public class TestGOAPNode
    {
        //[Test]
        public void ShouldEqualSameState()
        {
            
        }
    }
}