package org.tahomarobotics.robot.indexer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;

public class IndexerCommands {
    public static Pair<Command, Command> createIndexerCommands(Indexer indexer) {
        Command onTrue = indexer.runOnce(indexer::transitionToIndexing);
        Command onFalse = indexer.runOnce(indexer::transitionToDisabled);

        return Pair.of(onTrue, onFalse);
    }

    public static Pair<Command, Command> createIndexerEjectingCommands(Indexer indexer) {
        Command onTrue = indexer.runOnce(indexer::transitionToEjecting);
        Command onFalse = indexer.runOnce(indexer::transitionToDisabled);

        return Pair.of(onTrue, onFalse);
    }
}
