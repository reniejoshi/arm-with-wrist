package org.tahomarobotics.robot.util;

import org.tinylog.Logger;

import java.util.*;
import java.util.function.Function;

public class ImmutableLazyOptionalMap<K, V> {
    private final List<K> keys;
    private final Map<K, V> values = new HashMap<>();

    private final Function<K, Optional<V>> generator;

    public ImmutableLazyOptionalMap(List<K> keys, Function<K, Optional<V>> generator) {
        this.keys = new ArrayList<>(keys);
        this.generator = generator;
    }

    public Optional<V> get(K key) {
        if (!keys.contains(key)) {
            return Optional.empty();
        }

        if (!values.containsKey(key)) {
            Optional<V> value = generator.apply(key);
            if (value.isPresent()) {
                values.put(key, value.get());
                return value;
            } else {
                Logger.error("Initial key '{}' produced an invalid value! Removing from key list.", key);
                keys.remove(key);
                return Optional.empty();
            }
        } else {
            return Optional.of(values.get(key));
        }
    }

    public List<K> keys() {
        return List.copyOf(keys);
    }
}
