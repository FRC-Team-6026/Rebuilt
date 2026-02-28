package frc.lib.util;

import java.lang.reflect.Field;

public abstract class PrivateField {
    public static Object get(Object obj, String name) throws Exception {
        try {
            Field field = findField(obj.getClass(), name);
            field.setAccessible(true);
            return field.get(obj);
        } catch (Exception e) {
            throw new Exception(e);
        }
    }

    private static Field findField(Class<?> clazz, String name) throws NoSuchFieldException {
        while (clazz != null) {
            try {
                return clazz.getDeclaredField(name);
            } catch (NoSuchFieldException ignored) {
                clazz = clazz.getSuperclass();
            }
        }

        throw new NoSuchFieldException(name);
    }
}
