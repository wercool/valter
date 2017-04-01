#!/usr/bin/env python
# Script that rmeove observations objects from the database

from __future__ import print_function
import object_recognition_core.db.tools as dbtools
import couchdb
import argparse

def deletion_view(db):
    view = couchdb.design.ViewDefinition('object_recognition', 'refer_object_id', '''function(doc) {
         if (doc.object_id)
             emit(doc.object_id, doc);
    }''')
    view.sync(db)

def delete_object(db):
    for row in db.view('object_recognition/refer_object_id'):
        if row.value['Type'] == 'Observation':
          print('Deleting doc: %s of type "%s" referring to object: %s' % (row.id, row.value['Type'], row.key))
          del db[row.id]

if __name__ == "__main__":
    couch = couchdb.Server()
    db = dbtools.init_object_databases(couch)
    deletion_view(db)
    delete_object(db)

